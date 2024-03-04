const logoutButton = $("#logout-btn");
const profileName = $("#profile-name");
const dropDownSelections = $("#drop-down-selections");

/**
 * drop down the nav menu for when you are logged in
 */
profileName.click(() => {
    if (!dropDownSelections.hasClass("visible")) {
        dropDownSelections.addClass("visible");
        return;
    }
    dropDownSelections.removeClass("visible");
});

/**
 * The JavaScript AJAX call for when a user logs out of their account
 */
logoutButton.click((e) => {
    e.preventDefault();
    $.ajax({
        type: "POST",
        url: "/api/v1/logout",
        success: (res) => {
            if (res === 200) {
                window.location.href = "/";
            }
        },
        error: (error) => {
            setToast("error", "Something went wrong", "Cannot logout at this time");
            console.error(error);
        }
    });
});