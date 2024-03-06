const logoutButton = $("#logout-btn");


/**
 * The JavaScript AJAX call for when a user logs out of their account.
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